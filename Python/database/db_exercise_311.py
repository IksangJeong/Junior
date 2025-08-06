"""
데이터베이스 연습 #311
SQLite CRUD 실습
"""
import sqlite3

def setup_311():
    conn = sqlite3.connect(':memory:')
    cursor = conn.cursor()
    cursor.execute('''CREATE TABLE items_311 (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        name TEXT NOT NULL,
        value REAL,
        category TEXT
    )''')

    data = [
        [('item_311_0', 510.43, 'C'), ('item_311_1', 541.29, 'A'), ('item_311_2', 334.84, 'C'), ('item_311_3', 447.42, 'C'), ('item_311_4', 213.35, 'C'), ('item_311_5', 493.94, 'C')]
    ]
    cursor.executemany(
        'INSERT INTO items_311 (name, value, category) VALUES (?, ?, ?)',
        data
    )
    conn.commit()
    return conn

if __name__ == '__main__':
    conn = setup_311()
    cursor = conn.cursor()

    print(f"DB 연습 #311")
    cursor.execute('SELECT * FROM items_311')
    for row in cursor.fetchall():
        print(f"  {row}")

    cursor.execute('SELECT category, COUNT(*), AVG(value) FROM items_311 GROUP BY category')
    print("\n카테고리별 통계:")
    for row in cursor.fetchall():
        print(f"  {row[0]}: {row[1]}개, 평균 {row[2]:.2f}")

    conn.close()
