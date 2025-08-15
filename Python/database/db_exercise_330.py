"""
데이터베이스 연습 #330
SQLite CRUD 실습
"""
import sqlite3

def setup_330():
    conn = sqlite3.connect(':memory:')
    cursor = conn.cursor()
    cursor.execute('''CREATE TABLE items_330 (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        name TEXT NOT NULL,
        value REAL,
        category TEXT
    )''')

    data = [
        [('item_330_0', 590.02, 'B'), ('item_330_1', 94.84, 'C'), ('item_330_2', 302.43, 'A'), ('item_330_3', 715.98, 'A'), ('item_330_4', 275.29, 'B')]
    ]
    cursor.executemany(
        'INSERT INTO items_330 (name, value, category) VALUES (?, ?, ?)',
        data
    )
    conn.commit()
    return conn

if __name__ == '__main__':
    conn = setup_330()
    cursor = conn.cursor()

    print(f"DB 연습 #330")
    cursor.execute('SELECT * FROM items_330')
    for row in cursor.fetchall():
        print(f"  {row}")

    cursor.execute('SELECT category, COUNT(*), AVG(value) FROM items_330 GROUP BY category')
    print("\n카테고리별 통계:")
    for row in cursor.fetchall():
        print(f"  {row[0]}: {row[1]}개, 평균 {row[2]:.2f}")

    conn.close()
