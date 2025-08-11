"""
데이터베이스 연습 #323
SQLite CRUD 실습
"""
import sqlite3

def setup_323():
    conn = sqlite3.connect(':memory:')
    cursor = conn.cursor()
    cursor.execute('''CREATE TABLE items_323 (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        name TEXT NOT NULL,
        value REAL,
        category TEXT
    )''')

    data = [
        [('item_323_0', 797.03, 'A'), ('item_323_1', 148.56, 'C'), ('item_323_2', 493.43, 'C'), ('item_323_3', 990.42, 'A'), ('item_323_4', 528.7, 'A'), ('item_323_5', 945.74, 'C'), ('item_323_6', 58.03, 'A'), ('item_323_7', 418.27, 'C')]
    ]
    cursor.executemany(
        'INSERT INTO items_323 (name, value, category) VALUES (?, ?, ?)',
        data
    )
    conn.commit()
    return conn

if __name__ == '__main__':
    conn = setup_323()
    cursor = conn.cursor()

    print(f"DB 연습 #323")
    cursor.execute('SELECT * FROM items_323')
    for row in cursor.fetchall():
        print(f"  {row}")

    cursor.execute('SELECT category, COUNT(*), AVG(value) FROM items_323 GROUP BY category')
    print("\n카테고리별 통계:")
    for row in cursor.fetchall():
        print(f"  {row[0]}: {row[1]}개, 평균 {row[2]:.2f}")

    conn.close()
